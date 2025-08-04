"""
데이터베이스 연습 #308
SQLite CRUD 실습
"""
import sqlite3

def setup_308():
    conn = sqlite3.connect(':memory:')
    cursor = conn.cursor()
    cursor.execute('''CREATE TABLE items_308 (
        id INTEGER PRIMARY KEY AUTOINCREMENT,
        name TEXT NOT NULL,
        value REAL,
        category TEXT
    )''')

    data = [
        [('item_308_0', 981.87, 'B'), ('item_308_1', 330.37, 'A'), ('item_308_2', 16.76, 'B'), ('item_308_3', 243.17, 'B'), ('item_308_4', 954.49, 'B'), ('item_308_5', 586.64, 'C'), ('item_308_6', 21.32, 'C'), ('item_308_7', 365.96, 'A')]
    ]
    cursor.executemany(
        'INSERT INTO items_308 (name, value, category) VALUES (?, ?, ?)',
        data
    )
    conn.commit()
    return conn

if __name__ == '__main__':
    conn = setup_308()
    cursor = conn.cursor()

    print(f"DB 연습 #308")
    cursor.execute('SELECT * FROM items_308')
    for row in cursor.fetchall():
        print(f"  {row}")

    cursor.execute('SELECT category, COUNT(*), AVG(value) FROM items_308 GROUP BY category')
    print("\n카테고리별 통계:")
    for row in cursor.fetchall():
        print(f"  {row[0]}: {row[1]}개, 평균 {row[2]:.2f}")

    conn.close()
