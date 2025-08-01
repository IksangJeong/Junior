"""
데이터베이스 연습 #299
SQLite CRUD 실습
"""
import sqlite3

def setup_299():
    conn = sqlite3.connect(':memory:')
    cursor = conn.cursor()
    cursor.execute('''CREATE TABLE items_299 (
        id INTEGER PRIMARY KEY AUTOINCREMENT,
        name TEXT NOT NULL,
        value REAL,
        category TEXT
    )''')

    data = [
        [('item_299_0', 600.94, 'B'), ('item_299_1', 967.4, 'C'), ('item_299_2', 581.86, 'C'), ('item_299_3', 60.84, 'B'), ('item_299_4', 178.81, 'C'), ('item_299_5', 723.97, 'B'), ('item_299_6', 156.3, 'C'), ('item_299_7', 249.77, 'C')]
    ]
    cursor.executemany(
        'INSERT INTO items_299 (name, value, category) VALUES (?, ?, ?)',
        data
    )
    conn.commit()
    return conn

if __name__ == '__main__':
    conn = setup_299()
    cursor = conn.cursor()

    print(f"DB 연습 #299")
    cursor.execute('SELECT * FROM items_299')
    for row in cursor.fetchall():
        print(f"  {row}")

    cursor.execute('SELECT category, COUNT(*), AVG(value) FROM items_299 GROUP BY category')
    print("\n카테고리별 통계:")
    for row in cursor.fetchall():
        print(f"  {row[0]}: {row[1]}개, 평균 {row[2]:.2f}")

    conn.close()
