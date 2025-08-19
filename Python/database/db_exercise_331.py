"""
데이터베이스 연습 #331
SQLite CRUD 실습
"""
import sqlite3

def setup_331():
    conn = sqlite3.connect(':memory:')
    cursor = conn.cursor()
    cursor.execute('''CREATE TABLE items_331 (
        id INTEGER PRIMARY KEY AUTOINCREMENT,
        name TEXT NOT NULL,
        value REAL,
        category TEXT
    )''')

    data = [
        [('item_331_0', 323.51, 'A'), ('item_331_1', 100.61, 'B'), ('item_331_2', 450.29, 'B'), ('item_331_3', 111.21, 'A'), ('item_331_4', 195.52, 'B'), ('item_331_5', 561.42, 'C')]
    ]
    cursor.executemany(
        'INSERT INTO items_331 (name, value, category) VALUES (?, ?, ?)',
        data
    )
    conn.commit()
    return conn

if __name__ == '__main__':
    conn = setup_331()
    cursor = conn.cursor()

    print(f"DB 연습 #331")
    cursor.execute('SELECT * FROM items_331')
    for row in cursor.fetchall():
        print(f"  {row}")

    cursor.execute('SELECT category, COUNT(*), AVG(value) FROM items_331 GROUP BY category')
    print("\n카테고리별 통계:")
    for row in cursor.fetchall():
        print(f"  {row[0]}: {row[1]}개, 평균 {row[2]:.2f}")

    conn.close()
